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
#define PMIC_SCRATCH_PAD_REG_MIN ((uint8_t)PMIC_SCRATCH_PAD_REG_1)
#define PMIC_SCRATCH_PAD_REG_MAX ((uint8_t)PMIC_SCRATCH_PAD_REG_4)
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
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-523, PMICDRV-545, PMICDRV-546
 *
 * @param handle Pointer to the PMIC handle.
 *
 * @param lock True to lock the registers, false to unlock.
 *
 * @return PMIC_ST_SUCCESS if PMIC registers have been locked/unlocked, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setRegLockState(const Pmic_Handle_t *handle, bool lock);

/**
 * @brief Get PMIC register lock state.
 *
 * Design: PMICDRV-588
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-521, PMICDRV-522,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-546
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param isLocked [OUT] Register lock state obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if PMIC register lock state has been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getRegLockState(const Pmic_Handle_t *handle, bool *isLocked);

/**
 * @brief Get PMIC NVM revision from hardware register.
 *
 * Design: PMICDRV-584
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-524,
 *               PMICDRV-528, PMICDRV-547
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param nvmRev [OUT] PMIC NVM revision obtained from the device register.
 *
 * @return PMIC_ST_SUCCESS if the PMIC NVM revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getNvmRev(const Pmic_Handle_t *handle, uint8_t *nvmRev);

/**
 * @brief Get PMIC silicon revision from hardware register.
 *
 * Design: PMICDRV-759
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-521, PMICDRV-522, PMICDRV-524,
 *               PMICDRV-528, PMICDRV-547
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param siliconRev [OUT] PMIC silicon revision obtained from the device register.
 *
 * @return PMIC_ST_SUCCESS if the PMIC silicon revision has been obtained, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getSiliconRev(const Pmic_Handle_t *handle, uint8_t *siliconRev);

/**
 * @brief Set the value of a PMIC scratchpad register.
 *
 * Design: PMICDRV-591
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-523,
 *               PMICDRV-527, PMICDRV-545, PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Scratchpad register number. For valid
 * scratchpad register numbers, refer to @ref Pmic_ScratchpadRegs.
 *
 * @param value [IN] Value to be written to the scratchpad register.
 *
 * @return PMIC_ST_SUCCESS if the value has been written successfully,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t value);

/**
 * @brief Get the value of a PMIC scratchpad register.
 *
 * Design: PMICDRV-592
 * Architecture: PMICDRV-504, PMICDRV-506, PMICDRV-508, PMICDRV-511, PMICDRV-512,
 *               PMICDRV-515, PMICDRV-516, PMICDRV-521, PMICDRV-522, PMICDRV-527,
 *               PMICDRV-528, PMICDRV-545, PMICDRV-551
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param scratchPadRegNum [IN] Scratchpad register number. For valid
 * scratchpad register numbers, refer to @ref Pmic_ScratchpadRegs.
 *
 * @param value [OUT] Scratchpad register value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if the value has been read successfully,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getScratchPadValue(const Pmic_Handle_t *handle, uint8_t scratchPadRegNum, uint8_t *value);

/**
 * @brief Run CRC BIST, and depending on the input parameter, either update the
 * expected CRC value in the PMIC or check the CRC registers against the existing
 * expected CRC value.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param update [IN] If true, the expected register map CRC will be updated.
 * Otherwise, the PMIC will check the CRC registers against the existing expected
 * CRC value and report an error if there is a mismatch.
 *
 * @return PMIC_ST_SUCCESS if CRC BIST has been successfully triggered, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcRun(const Pmic_Handle_t *handle, bool update);

/**
 * @brief Write the full 16-bit configuration register CRC value to the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param value [IN] 16-bit CRC value to be written to the PMIC.
 *
 * @return PMIC_ST_SUCCESS if the CRC value has been successfully written, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_setConfigCrc(const Pmic_Handle_t *handle, uint16_t value);

/**
 * @brief Read the full 16-bit configuration register CRC value from the PMIC.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param value [OUT] 16-bit CRC value read from the PMIC.
 *
 * @return PMIC_ST_SUCCESS if the CRC value has been successfully read, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_getConfigCrc(const Pmic_Handle_t *handle, uint16_t *value);

/**
 * @brief Calculate the CRC-16 over the PMIC configuration registers
 * (0x000-0x0EF and 0x401-0x40A), write the result to
 * REGMAP_USER_CRC_HIGH_REG/REGMAP_USER_CRC_LOW_REG, and trigger a hardware
 * BIST to verify the stored value is correct.
 *
 * Polynomial : 0x755B (x^16+x^14+x^13+x^12+x^10+x^8+x^6+x^4+x^3+x+1)
 * Init       : 0xFFFF
 * Bit order  : big-endian (RefIn/RefOut = false)
 *
 * @note The hardware BIST result is reported via REG_CRC_ERR_INT in
 * INT_MODERATE_ERR. The interrupt latch is cleared before triggering the
 * BIST and re-read immediately after.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS if the CRC was calculated, stored, and verified
 * successfully. Returns PMIC_ST_ERR_CONFIG_REG_CRC if the hardware BIST
 * reports a mismatch after the CRC is written. For all valid codes, refer
 * to @ref Pmic_ErrorCodes.
 */
int32_t Pmic_configCrcCalculate(const Pmic_Handle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_CORE_H */
