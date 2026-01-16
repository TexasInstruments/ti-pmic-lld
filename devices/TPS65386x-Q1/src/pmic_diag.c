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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>

#include "pmic.h"
#include "pmic_common.h"
#include "pmic_diag.h"
#include "pmic_io.h"
#include "regmap/core.h"

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */

int32_t Pmic_diagSetOutCtrlCfg(const Pmic_Handle_t *handle, const Pmic_DiagOutCfgCtrl_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    uint8_t ctrlValue = 0U;

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);

        if (Pmic_validParamStatusCheck(config->validParams, PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID, status)) {
            if (config->diagOutCtrl_AMUXEn != 0U) {
                ctrlValue = PMIC_DIAG_OUT_CTRL_AMUX_VALUE;
            }
        }

        if (Pmic_validParamStatusCheck(config->validParams, PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID, status)) {
            if (config->diagOutCtrl_DMUXEn != 0U) {
                ctrlValue = PMIC_DIAG_OUT_CTRL_DMUX_VALUE;
            }
        }

        if (Pmic_validParamStatusCheck(config->validParams, PMIC_DIAG_OUT_CTRL_VALID, status)) {
            ctrlValue = config->diagOutCtrl;
        }

        if ((status == PMIC_ST_SUCCESS) &&
            ((config->validParams & (PMIC_DIAG_OUT_CTRL_AMUX_EN_VALID |
                                    PMIC_DIAG_OUT_CTRL_DMUX_EN_VALID |
                                    PMIC_DIAG_OUT_CTRL_VALID)) != 0U)) {
            Pmic_setBitField(&regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK, ctrlValue);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_diagGetOutCtrlCfg(const Pmic_Handle_t *handle, Pmic_DiagOutCfgCtrl_t *config) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (config == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        config->diagOutCtrl = Pmic_getBitField(regData, PMIC_DIAG_OUT_CTRL_SHIFT, PMIC_DIAG_OUT_CTRL_MASK);
        config->diagOutCtrl_AMUXEn = (config->diagOutCtrl == PMIC_DIAG_OUT_CTRL_AMUX_VALUE) ? 1U : 0U;
        config->diagOutCtrl_DMUXEn = (config->diagOutCtrl == PMIC_DIAG_OUT_CTRL_DMUX_VALUE) ? 1U : 0U;
    }

    return status;
}

int32_t Pmic_diagSetAmuxCfg(const Pmic_Handle_t *handle, uint8_t channel) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (channel > 0x1FU)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK, channel);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_diagGetAmuxCfg(const Pmic_Handle_t *handle, uint8_t *channel) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (channel == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *channel = Pmic_getBitField(regData, PMIC_DIAG_CH_SEL_SHIFT, PMIC_DIAG_CH_SEL_MASK);
    }

    return status;
}

int32_t Pmic_diagSetDmuxCfg(const Pmic_Handle_t *handle, uint8_t group) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (group > 0x1FU)) {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS) {
            Pmic_setBitField(&regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK, group);
            status = Pmic_ioTxByte(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_diagGetDmuxCfg(const Pmic_Handle_t *handle, uint8_t *group) {
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (group == NULL)) {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS) {
        status = Pmic_ioRxByte_CS(handle, PMIC_DIAG_OUT_CFG_CTRL_REG, &regData);
    }

    if (status == PMIC_ST_SUCCESS) {
        *group = Pmic_getBitField(regData, PMIC_DIAG_GRP_SEL_SHIFT, PMIC_DIAG_GRP_SEL_MASK);
    }

    return status;
}
