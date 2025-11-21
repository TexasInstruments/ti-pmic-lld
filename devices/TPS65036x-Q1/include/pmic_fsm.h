/******************************************************************************
 * Copyright (c) 2024 - 2025 Texas Instruments Incorporated - http://www.ti.com
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
 * @file pmic_fsm.h
 *
 * @brief PMIC LLD FSM module header file.
 *
 * @details User-facing FSM APIs.
 */
#ifndef __PMIC_GPIO_H__
#define __PMIC_GPIO_H__

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
 * @anchor Pmic_resetRecovCntThrMax
 * @name TPS65036x RESET_CNT and RECOV_CNT Maximum Threshold
 *
 * @brief Maximum thresholds of RESET_CNT and RECOV_CNT.
 *
 * @{
 */
#define PMIC_RESET_RECOV_CNT_THR_MAX        ((uint8_t)0xFU)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/* ========================================================================== */
/*                             Function Declarations                          */
/* ========================================================================== */

/**
 * @brief Set desired PMIC device state.
 *
 * Design: PMICDRV-601
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-540, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param fsmCmd [IN] MCU command for FSM state transition. For valid values,
 * refer to @ref Pmic_fsmCommands.
 *
 * @return Success code if the FSM command has been sent to the PMIC, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmSetDevState(const Pmic_Handle_t *pmicHandle, uint8_t fsmCmd);

/**
 * @brief Set PMIC recovery counter threshold.
 *
 * Design: PMICDRV-695
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-540, PMICDRV-521, PMICDRV-512
 *
 * @details The PMIC has a counter called RECOV_CNT (recovery counter) that
 * that is incremented each time the PMIC goes to SAFE state. If the counter
 * meets or exceeds the recovery counter threshold (RECOV_CNT >= RECOV_CNT_THR),
 * the PMIC stays in SAFE state until a power cycle occours.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param threshold [IN] Desired recovery counter threshold to be set. See
 * @ref Pmic_resetRecovCntThrMax for the maximum valid value.
 *
 * @return Success code if PMIC recovery counter threshold has been set, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmSetRecovCntThr(const Pmic_Handle_t *pmicHandle, uint8_t threshold);

/**
 * @brief Get PMIC recovery counter threshold.
 *
 * Design: PMICDRV-696
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-540, PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param threshold [OUT] Recovery counter threshold value obtained from PMIC.
 *
 * @return Success code if PMIC recovery counter threshold has been obtained,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmGetRecovCntThr(const Pmic_Handle_t *pmicHandle, uint8_t *threshold);

/**
 * @brief Get value of the PMIC recovery counter.
 *
 * Design: PMICDRV-693
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-540, PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param recovCnt [OUT] PMIC recovery counter value obtained from PMIC.
 *
 * @return Success code if PMIC recovery counter value has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmGetRecovCnt(const Pmic_Handle_t *pmicHandle, uint8_t *recovCnt);

/**
 * @brief Clear PMIC recovery counter.
 *
 * Design: PMICDRV-694
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-540, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC recovery counter has been cleared without issues,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmClrRecovCnt(const Pmic_Handle_t *pmicHandle);

/**
 * @brief Set PMIC reset counter threshold.
 *
 * Design: PMICDRV-740
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-540, PMICDRV-521, PMICDRV-512
 *
 * @details The PMIC has a counter called RESET_CNT (reset counter) that
 * increments each time the PMIC enters WARM RESET state. When the counter meets
 * or exceeds the reset counter threshold (RESET_CNT >= RESET_CNT_THR), the PMIC
 * executes an orderly shutdown, enters SAFE state, clears RESET_CNT, and
 * RESET_CNT_INT is asserted.
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param threshold [IN] Desired PMIC reset counter threshold to be set. See
 * @ref Pmic_resetRecovCntThrMax for the maximum valid value.
 *
 * @return Success code if PMIC reset counter threshold has been set, error code
 * otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmSetResetCntThr(const Pmic_Handle_t *pmicHandle, uint8_t threshold);

/**
 * @brief Get PMIC reset counter threshold.
 *
 * Design: PMICDRV-741
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-540, PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param threshold [OUT] PMIC reset counter threshold value obtained from the
 * PMIC.
 *
 * @return Success code if PMIC reset counter threshold has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmGetResetCntThr(const Pmic_Handle_t *pmicHandle, uint8_t *threshold);

/**
 * @brief Get value of the PMIC reset counter.
 *
 * Design: PMICDRV-742
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-551, PMICDRV-506, PMICDRV-504
 *               PMICDRV-522, PMICDRV-540, PMICDRV-528, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @param resetCnt [OUT] PMIC reset counter value obtained from PMIC.
 *
 * @return Success code if PMIC reset counter value has been obtained, error
 * code otherwise. For valid success/error codes, refer to @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmGetResetCnt(const Pmic_Handle_t *pmicHandle, uint8_t *resetCnt);

/**
 * @brief Clear PMIC reset counter.
 *
 * Design: PMICDRV-743
 * Architecture: PMICDRV-507, PMICDRV-516, PMICDRV-508, PMICDRV-523, PMICDRV-551, PMICDRV-506
 *               PMICDRV-504, PMICDRV-522, PMICDRV-540, PMICDRV-521, PMICDRV-512
 *
 * @param pmicHandle [IN] PMIC interface handle.
 *
 * @return Success code if PMIC reset counter has been cleared without issues,
 * error code otherwise. For valid success/error codes, refer to
 * @ref Pmic_errorCodes.
 */
int32_t Pmic_fsmClrResetCnt(const Pmic_Handle_t *pmicHandle);

#ifdef __cplusplus
}

#endif /* __cplusplus */
#endif /* __PMIC_FSM_H__ */
