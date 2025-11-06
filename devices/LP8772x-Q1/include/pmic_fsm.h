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
 * @file pmic_fsm.h
 *
 * @brief PMIC LLD FSM module header file.
 *
 * @details This module contains declarations/definitions of macros, data
 * structures, and APIs used to interact with the PMIC FSM functionality.
 */
#ifndef PMIC_FSM_H
#define PMIC_FSM_H

#ifdef __cplusplus
extern "C" {
#endif

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */

#include <stdint.h>
#include <stdbool.h>

#include "pmic_common.h"

/* ========================================================================== */
/*                               Macros & Typedefs                            */
/* ========================================================================== */

/**
 * @anchor Pmic_FsmMcuCommands
 * @name PMIC FSM MCU Commands
 *
 * @brief MCU commands for FSM state transitions
 *
 * @{
 */
#define PMIC_FSM_COMMAND_OFF_REQ               (0x99U)
#define PMIC_FSM_COMMAND_COLD_BOOT_REQ         (0x55U)
#define PMIC_FSM_COMMAND_WARM_RESET_REQ        (0xCCU)
#define PMIC_FSM_COMMAND_SAFE_RECOV_REQ        (0x4BU)
#define PMIC_FSM_COMMAND_OTA_FW_DOWNLOAD_REQ   (0x87U)
/** @} */

/**
 * @anchor Pmic_FsmResetRecovCntThrMax
 * @name PMIC FSM Reset/Recovery Counter Threshold
 *
 * @brief Maximum value for the reset/recovery counter threshold
 *
 * @{
 */
#define PMIC_FSM_RESET_RECOV_CNT_THR_MAX       (0x0FU)
/** @} */

/* ========================================================================== */
/*                             Structures and Enums                           */
/* ========================================================================== */

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Set desired PMIC device state.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param cmd    [IN] Command request to perform, see @ref Pmic_FsmMcuCommands.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetDevState(Pmic_CoreHandle_t *handle, uint8_t cmd);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Set PMIC reset counter threshold (RESET_CNT_THR).
 *
 * @details Each time the PMIC goes through Warm Reset, the reset counter
 * (RESET_CNT) increments. When RESET_CNT exceeds RESET_CNT_THR - the value in
 * which this API configures - the PMIC power downs all its rails.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param resetCntThr [IN] Desired reset counter threshold to set. For the
 * maximum value, refer to @ref Pmic_FsmResetRecovCntThrMax.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetResetCntThr(Pmic_CoreHandle_t *handle, uint8_t resetCntThr);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Get PMIC reset counter threshold (RESET_CNT_THR).
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param resetCntThr [OUT] Reset counter threshold value obtained from the
 * PMIC.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetResetCntThr(Pmic_CoreHandle_t *handle, uint8_t *resetCntThr);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Get PMIC reset counter.
 *
 * @details The PMIC increments the reset counter - the value in which this
 * API obtains - each time it goes through warm reset.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param resetCnt [OUT] Reset counter value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetResetCnt(Pmic_CoreHandle_t *handle, uint8_t *resetCnt);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Clear PMIC reset counter to zero.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmClrResetCnt(Pmic_CoreHandle_t *handle);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Set PMIC recovery counter threshold.
 *
 * @details The PMIC increments the recovery counter (RECOV_CNT) when it
 * transitions from ACTIVE to SAFE state. When RECOV_CNT exceeds
 * RECOV_CNT_THR - the value in which this API configures - the PMIC stays in
 * SAFE state until supply power cycle occurs.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCntThr [IN] Desired recovery counter threshold value to be set.
 * For the maximum value, refer to @ref Pmic_FsmResetRecovCntThrMax.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmSetRecovCntThr(Pmic_CoreHandle_t *handle, uint8_t recovCntThr);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Get PMIC recovery counter threshold.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCntThr [OUT] Recovery counter threshold value obtained from the
 * PMIC.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetRecovCntThr(Pmic_CoreHandle_t *handle, uint8_t *recovCntThr);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Get PMIC recovery counter.
 *
 * @details The PMIC increments the recovery counter - the value in which this
 * API obtains - each time it transitions from ACTIVE to SAFE state.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @param recovCnt [OUT] Recovery counter value obtained from the PMIC.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmGetRecovCnt(Pmic_CoreHandle_t *handle, uint8_t *recovCnt);

/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief Clear PMIC recovery counter.
 *
 * @param handle [IN] PMIC interface handle.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmClrRecovCnt(Pmic_CoreHandle_t *handle);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* PMIC_FSM_H */
