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

#ifndef __PMIC_FSM_H__
#define __PMIC_FSM_H__

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

/* ========================================================================== */
/*                            Function Declarations                           */
/* ========================================================================== */
/**
 * @ingroup DRV_PMIC_FSM_CONFIG_GROUP
 * @brief API to perform an MCU command request.
 *
 * This function is used to perform one of the supported MCU command requests
 * which can command PMIC FSM state transitions.
 *
 * @param handle [IN] PMIC Interface Handle
 * @param cmd    [IN] Command request to perform, see @ref Pmic_FsmMcuCommands.
 *
 * @return PMIC_ST_SUCCESS in case of success or appropriate error code. For
 * possible values @ref Pmic_ErrorCodes
 */
int32_t Pmic_fsmMcuCommand(Pmic_CoreHandle_t *handle, uint8_t cmd);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif /* __PMIC_FSM_H__ */
