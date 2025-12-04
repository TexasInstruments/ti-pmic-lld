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
/**
 * @file pmic_esm.c
 *
 * @brief ESM module API definitions.
 */

/* ========================================================================== */
/*                              Include Files                                 */
/* ========================================================================== */

#include "pmic.h"

#include "regmap/esm.h"

/* ========================================================================== */
/*                            Macros & Typedefs                               */
/* ========================================================================== */

/* ========================================================================== */
/*                           Function Definitions                             */
/* ========================================================================== */

int32_t Pmic_esmSetStartState(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioUpdateByte_bCS(handle, ESM_START_REG, ESM_MCU_START_SHIFT, enable);
}

int32_t Pmic_esmGetStartState(const Pmic_Handle_t *handle, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte_CS(handle, ESM_START_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isEnabled = Pmic_getBitField_b(regData, ESM_MCU_START_SHIFT);
    return PMIC_ST_SUCCESS;
}

int32_t Pmic_esmSetEnableState(const Pmic_Handle_t *handle, bool enable) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioUpdateByte_bCS(handle, ESM_MODE_CFG_REG, ESM_MCU_EN_SHIFT, enable);
}

int32_t Pmic_esmGetEnableState(const Pmic_Handle_t *handle, bool *isEnabled) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte_CS(handle, ESM_MODE_CFG_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *isEnabled = Pmic_getBitField_b(regData, ESM_MCU_EN_SHIFT);
    return PMIC_ST_SUCCESS;
}

static int32_t ESM_setDelays(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY1_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_DELAY1_REG, esmCfg->delay1);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY2_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_DELAY2_REG, esmCfg->delay2);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t ESM_setHighLowThresholds(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg) {
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_HMAX_REG, esmCfg->hmax);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_HMIN_REG, esmCfg->hmin);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_LMAX_REG, esmCfg->lmax);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID)) {
        status = Pmic_ioTxByte_CS(handle, ESM_LMIN_REG, esmCfg->lmin);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t ESM_setOtherCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    Pmic_criticalSectionStart(handle);
    status = Pmic_ioRxByte(handle, ESM_MODE_CFG_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MODE_VALID)) {
        if (esmCfg->mode > PMIC_ESM_MODE_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, ESM_MCU_MODE_SHIFT, ESM_MCU_MODE_MASK, esmCfg->mode);
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ERR_CNT_THR_VALID)) {
        if (esmCfg->errCntThr > PMIC_ESM_ERR_CNT_THR_MAX) {
            return PMIC_ST_ERR_INV_PARAM;
        } else {
            Pmic_setBitField(&regData, ESM_MCU_ERR_CNT_TH_SHIFT, ESM_MCU_ERR_CNT_TH_MASK, esmCfg->errCntThr);
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID)) {
        Pmic_setBitField_b(&regData, ESM_MCU_CAN_DIS_SHIFT, esmCfg->disableCanOnFault);
    }

    status = Pmic_ioTxByte(handle, ESM_MODE_CFG_REG, regData);
    Pmic_criticalSectionStop(handle);

    return status;
}

int32_t Pmic_esmSetCfg(const Pmic_Handle_t *handle, const Pmic_EsmCfg_t *esmCfg) {
    const uint32_t delayValid = (PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID);
    const uint32_t highLowThresholdValid = (PMIC_ESM_HMAX_VALID | PMIC_ESM_HMIN_VALID | PMIC_ESM_LMAX_VALID | PMIC_ESM_LMIN_VALID);
    const uint32_t otherCfgValid = (PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID | PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID);

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (esmCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (esmCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(esmCfg->validParams, delayValid)) {
        status = ESM_setDelays(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, highLowThresholdValid)) {
        status = ESM_setHighLowThresholds(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, otherCfgValid)) {
        status = ESM_setOtherCfg(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t ESM_getDelays(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY1_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_DELAY1_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->delay1 = regData;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DELAY2_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_DELAY2_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->delay2 = regData;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t ESM_getHighLowThresholds(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg) {
    uint8_t regData = 0U;
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMAX_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_HMAX_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->hmax = regData;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_HMIN_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_HMIN_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->hmin = regData;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMAX_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_LMAX_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->lmax = regData;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_LMIN_VALID)) {
        status = Pmic_ioRxByte_CS(handle, ESM_LMIN_REG, &regData);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        } else {
            esmCfg->lmin = regData;
        }
    }

    return PMIC_ST_SUCCESS;
}

static int32_t ESM_getOtherCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg) {
    uint8_t regData = 0U;
    int32_t status = Pmic_ioRxByte_CS(handle, ESM_MODE_CFG_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_MODE_VALID)) {
        esmCfg->mode = Pmic_getBitField(regData, ESM_MCU_MODE_SHIFT, ESM_MCU_MODE_MASK);
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_ERR_CNT_THR_VALID)) {
        esmCfg->errCntThr = Pmic_getBitField(regData, ESM_MCU_ERR_CNT_TH_SHIFT, ESM_MCU_ERR_CNT_TH_MASK);
    }

    if (Pmic_validParamCheck(esmCfg->validParams, PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID)) {
        esmCfg->disableCanOnFault = Pmic_getBitField_b(regData, ESM_MCU_CAN_DIS_SHIFT);
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_esmGetCfg(const Pmic_Handle_t *handle, Pmic_EsmCfg_t *esmCfg) {
    const uint32_t delayValid = (PMIC_ESM_DELAY1_VALID | PMIC_ESM_DELAY2_VALID);
    const uint32_t highLowThresholdValid = (PMIC_ESM_HMAX_VALID | PMIC_ESM_HMIN_VALID | PMIC_ESM_LMAX_VALID | PMIC_ESM_LMIN_VALID);
    const uint32_t otherCfgValid = (PMIC_ESM_MODE_VALID | PMIC_ESM_ERR_CNT_THR_VALID | PMIC_ESM_DISABLE_CAN_ON_FAULT_VALID);

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (esmCfg == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (esmCfg->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(esmCfg->validParams, delayValid)) {
        status = ESM_getDelays(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, highLowThresholdValid)) {
        status = ESM_getHighLowThresholds(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    if (Pmic_validParamCheck(esmCfg->validParams, otherCfgValid)) {
        status = ESM_getOtherCfg(handle, esmCfg);
        if (status != PMIC_ST_SUCCESS) {
            return status;
        }
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_esmClrErrStatus(const Pmic_Handle_t *handle, const Pmic_EsmErrStatus_t *errStatus) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (errStatus == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (errStatus->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_RST_INT_VALID)) {
        Pmic_setBitField_b(&regData, ESM_MCU_RST_INT_SHIFT, errStatus->rstInt);
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_FAIL_INT_VALID)) {
        Pmic_setBitField_b(&regData, ESM_MCU_FAIL_INT_SHIFT, errStatus->failInt);
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_PIN_INT_VALID)) {
        Pmic_setBitField_b(&regData, ESM_MCU_PIN_INT_SHIFT, errStatus->pinInt);
    }

    return Pmic_ioTxByte_CS(handle, INT_ESM_REG, regData);
}

int32_t Pmic_esmClrErrStatusAll(const Pmic_Handle_t *handle) {
    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    return Pmic_ioTxByte_CS(handle, INT_ESM_REG, 0xFFU);
}

int32_t Pmic_esmGetErrStatus(const Pmic_Handle_t *handle, Pmic_EsmErrStatus_t *errStatus) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (errStatus == NULL) {
        return PMIC_ST_ERR_NULL_PARAM;
    }

    if (errStatus->validParams == 0U) {
        return PMIC_ST_ERR_INV_PARAM;
    }

    status = Pmic_ioRxByte_CS(handle, INT_ESM_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_RST_INT_VALID)) {
        errStatus->rstInt = Pmic_getBitField_b(regData, ESM_MCU_RST_INT_SHIFT);
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_FAIL_INT_VALID)) {
        errStatus->failInt = Pmic_getBitField_b(regData, ESM_MCU_FAIL_INT_SHIFT);
    }

    if (Pmic_validParamCheck(errStatus->validParams, PMIC_ESM_PIN_INT_VALID)) {
        errStatus->pinInt = Pmic_getBitField_b(regData, ESM_MCU_PIN_INT_SHIFT);
    }

    return PMIC_ST_SUCCESS;
}

int32_t Pmic_esmGetErrCnt(const Pmic_Handle_t *handle, uint8_t *errCnt) {
    uint8_t regData = 0U;

    int32_t status = Pmic_checkHandle(handle);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    status = Pmic_ioRxByte_CS(handle, ESM_ERR_CNT_REG, &regData);
    if (status != PMIC_ST_SUCCESS) {
        return status;
    }

    *errCnt = Pmic_getBitField(regData, ESM_MCU_ERR_CNT_SHIFT, ESM_MCU_ERR_CNT_MASK);
    return PMIC_ST_SUCCESS;
}
