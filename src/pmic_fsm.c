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

/* ========================================================================== */
/*                                Include Files                               */
/* ========================================================================== */
#include <stdint.h>
#include <stdbool.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_io.h"

#include "pmic_fsm.h"
#include "regmap/fsm.h"

/* ========================================================================== */
/*                          Function Definitions                              */
/* ========================================================================== */
static inline bool FSM_assertMcuCommandValid(uint8_t cmd)
{
    bool commandValid = false;

    const uint8_t validCommands[] = {
        PMIC_FSM_COMMAND_OFF_REQ,
        PMIC_FSM_COMMAND_COLD_BOOT_REQ,
        PMIC_FSM_COMMAND_WARM_RESET_REQ,
        PMIC_FSM_COMMAND_SAFE_RECOV_REQ,
        PMIC_FSM_COMMAND_OTA_FW_DOWNLOAD_REQ,
    };

    for (uint8_t i = 0U; i < COUNT(validCommands); i++) {
        if (cmd == validCommands[i]) {
            commandValid = true;
            break;
        }
    }

    return commandValid;
}

int32_t Pmic_fsmMcuCommand(Pmic_CoreHandle_t *handle, uint8_t cmd)
{
    int32_t status = Pmic_checkPmicCoreHandle(handle);

    // validate that the requested command is supported, otherwise return error
    if ((status == PMIC_ST_SUCCESS) && (FSM_assertMcuCommandValid(cmd) == false)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle);
        status = Pmic_ioTxByte(handle, FSM_COMMAND_REG, cmd);
        Pmic_criticalSectionStop(handle);
    }

    return status;
}
