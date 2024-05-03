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
 *   @file    pmic_ilim.c
 *
 *   @brief   This file contains the default API's for PMIC ILIM configuration
 */
#include "pmic_ilim.h"
#include "pmic_ilim_priv.h"

/**
 * @brief API to set ILIM Configuration
 */
int32_t Pmic_SetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ILIMConfig_t *pPmicILIMConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t cfgReg = 0U;
    uint8_t degReg = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, &cfgReg);

    if (pmicStatus == PMIC_ST_SUCCESS) {
        pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_DGL_CFG_REGADDR, &degReg);
    }

    if (pmicStatus == PMIC_ST_SUCCESS) {
        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_LDO1_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_LDO1_ILIM_CFG_SHIFT, PMIC_LDO1_ILIM_CFG_MASK, pPmicILIMConfig->ldo1.errReact);
            Pmic_setBitField(&degReg, PMIC_LDO1_ILIM_DGL_CFG_SHIFT, PMIC_LDO1_ILIM_DGL_CFG_MASK, pPmicILIMConfig->ldo1.deglitch);
        }

        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_LDO2_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_LDO2_ILIM_CFG_SHIFT, PMIC_LDO2_ILIM_CFG_MASK, pPmicILIMConfig->ldo2.errReact);
            Pmic_setBitField(&degReg, PMIC_LDO2_ILIM_DGL_CFG_SHIFT, PMIC_LDO2_ILIM_DGL_CFG_MASK, pPmicILIMConfig->ldo2.deglitch);
        }

        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_LDO3_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_LDO3_ILIM_CFG_SHIFT, PMIC_LDO3_ILIM_CFG_MASK, pPmicILIMConfig->ldo3.errReact);
            Pmic_setBitField(&degReg, PMIC_LDO3_ILIM_DGL_CFG_SHIFT, PMIC_LDO3_ILIM_DGL_CFG_MASK, pPmicILIMConfig->ldo3.deglitch);
        }

        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_LDO4_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_LDO4_ILIM_CFG_SHIFT, PMIC_LDO4_ILIM_CFG_MASK, pPmicILIMConfig->ldo4.errReact);
            Pmic_setBitField(&degReg, PMIC_LDO4_ILIM_DGL_CFG_SHIFT, PMIC_LDO4_ILIM_DGL_CFG_MASK, pPmicILIMConfig->ldo4.deglitch);
        }

        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_PLDO1_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_PLDO1_ILIM_CFG_SHIFT, PMIC_PLDO1_ILIM_CFG_MASK, pPmicILIMConfig->pldo1.errReact);
            Pmic_setBitField(&degReg, PMIC_PLDO1_ILIM_DGL_CFG_SHIFT, PMIC_PLDO1_ILIM_DGL_CFG_MASK, pPmicILIMConfig->pldo1.deglitch);
        }

        if (Pmic_validParamCheck(pPmicILIMConfig->validParams, PMIC_ILIM_PLDO2_VALID)) {
            Pmic_setBitField(&cfgReg, PMIC_PLDO2_ILIM_CFG_SHIFT, PMIC_PLDO2_ILIM_CFG_MASK, pPmicILIMConfig->pldo2.errReact);
            Pmic_setBitField(&degReg, PMIC_PLDO2_ILIM_DGL_CFG_SHIFT, PMIC_PLDO2_ILIM_DGL_CFG_MASK, pPmicILIMConfig->pldo2.deglitch);
        }
    }

    if (pmicStatus == PMIC_ST_SUCCESS) {
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, cfgReg);
    }

    if (pmicStatus == PMIC_ST_SUCCESS) {
        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, degReg);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to clear ILIM STAT Configuration
 */
int32_t Pmic_ClearILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle,
                              Pmic_ILIMStatus_t *pPmicILIMStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_LDO1_VALID)) {
            Pmic_setBitField(& regData, PMIC_LDO1_ILIM_ERR_SHIFT, PMIC_LDO1_ILIM_ERR_MASK, pPmicILIMStat->ldo1);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_LDO2_VALID)) {
            Pmic_setBitField(&regData, PMIC_LDO2_ILIM_ERR_SHIFT, PMIC_LDO2_ILIM_ERR_MASK, pPmicILIMStat->ldo2);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_LDO3_VALID)) {
            Pmic_setBitField(&regData, PMIC_LDO3_ILIM_ERR_SHIFT, PMIC_LDO3_ILIM_ERR_MASK, pPmicILIMStat->ldo3);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_LDO4_VALID)) {
            Pmic_setBitField(&regData, PMIC_LDO4_ILIM_ERR_SHIFT, PMIC_LDO4_ILIM_ERR_MASK, pPmicILIMStat->ldo4);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_PLDO1_VALID)) {
            Pmic_setBitField(&regData, PMIC_PLDO1_ILIM_ERR_SHIFT, PMIC_PLDO1_ILIM_ERR_MASK, pPmicILIMStat->pldo1);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_PLDO2_VALID)) {
            Pmic_setBitField(&regData, PMIC_PLDO2_ILIM_ERR_SHIFT, PMIC_PLDO2_ILIM_ERR_MASK, pPmicILIMStat->pldo2);
        }

        if (Pmic_validParamCheck(pPmicILIMStat->validParams, PMIC_ILIM_BB_AVG_VALID)) {
            Pmic_setBitField(&regData, PMIC_BB_AVG_ILIM_ERR_SHIFT, PMIC_BB_AVG_ILIM_ERR_MASK, pPmicILIMStat->bbAvg);
        }

        pmicStatus = Pmic_commIntf_sendByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR, regData);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to get ILIM Configuration
 */
int32_t Pmic_GetILIMConfig(Pmic_CoreHandle_t *pPmicCoreHandle,
                           Pmic_ILIMConfig_t *pPmicILIMConfig) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t cfgReg = 0U;
    uint8_t degReg = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_CFG_REGADDR, &cfgReg);
    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_DGL_CFG_REGADDR, &degReg);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicILIMConfig->ldo1.errReact = Pmic_getBitField(cfgReg, PMIC_LDO1_ILIM_CFG_SHIFT, PMIC_LDO1_ILIM_CFG_MASK);
        pPmicILIMConfig->ldo1.deglitch = Pmic_getBitField(degReg, PMIC_LDO1_ILIM_DGL_CFG_SHIFT, PMIC_LDO1_ILIM_DGL_CFG_MASK);

        pPmicILIMConfig->ldo2.errReact = Pmic_getBitField(cfgReg, PMIC_LDO2_ILIM_CFG_SHIFT, PMIC_LDO2_ILIM_CFG_MASK);
        pPmicILIMConfig->ldo2.deglitch = Pmic_getBitField(degReg, PMIC_LDO2_ILIM_DGL_CFG_SHIFT, PMIC_LDO2_ILIM_DGL_CFG_MASK);

        pPmicILIMConfig->ldo3.errReact = Pmic_getBitField(cfgReg, PMIC_LDO3_ILIM_CFG_SHIFT, PMIC_LDO3_ILIM_CFG_MASK);
        pPmicILIMConfig->ldo3.deglitch = Pmic_getBitField(degReg, PMIC_LDO3_ILIM_DGL_CFG_SHIFT, PMIC_LDO3_ILIM_DGL_CFG_MASK);

        pPmicILIMConfig->ldo4.errReact = Pmic_getBitField(cfgReg, PMIC_LDO4_ILIM_CFG_SHIFT, PMIC_LDO4_ILIM_CFG_MASK);
        pPmicILIMConfig->ldo4.deglitch = Pmic_getBitField(degReg, PMIC_LDO4_ILIM_DGL_CFG_SHIFT, PMIC_LDO4_ILIM_DGL_CFG_MASK);

        pPmicILIMConfig->pldo1.errReact = Pmic_getBitField(cfgReg, PMIC_PLDO1_ILIM_CFG_SHIFT, PMIC_PLDO1_ILIM_CFG_MASK);
        pPmicILIMConfig->pldo1.deglitch = Pmic_getBitField(degReg, PMIC_PLDO1_ILIM_DGL_CFG_SHIFT, PMIC_PLDO1_ILIM_DGL_CFG_MASK);

        pPmicILIMConfig->pldo2.errReact = Pmic_getBitField(cfgReg, PMIC_PLDO2_ILIM_CFG_SHIFT, PMIC_PLDO2_ILIM_CFG_MASK);
        pPmicILIMConfig->pldo2.deglitch = Pmic_getBitField(degReg, PMIC_PLDO2_ILIM_DGL_CFG_SHIFT, PMIC_PLDO2_ILIM_DGL_CFG_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}

/**
 * @brief API to get ILIM STAT Configuration
 */
int32_t Pmic_GetILIMErrStat(Pmic_CoreHandle_t *pPmicCoreHandle, Pmic_ILIMStatus_t *pPmicILIMStat) {
    int32_t pmicStatus = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    Pmic_criticalSectionStart(pPmicCoreHandle);

    pmicStatus = Pmic_commIntf_recvByte(pPmicCoreHandle, PMIC_ILIM_STAT_REGADDR, &regData);

    if (PMIC_ST_SUCCESS == pmicStatus) {
        pPmicILIMStat->ldo1 = Pmic_getBitField(regData, PMIC_LDO1_ILIM_ERR_SHIFT, PMIC_LDO1_ILIM_ERR_MASK);
        pPmicILIMStat->ldo2 = Pmic_getBitField(regData, PMIC_LDO2_ILIM_ERR_SHIFT, PMIC_LDO2_ILIM_ERR_MASK);
        pPmicILIMStat->ldo3 = Pmic_getBitField(regData, PMIC_LDO3_ILIM_ERR_SHIFT, PMIC_LDO3_ILIM_ERR_MASK);
        pPmicILIMStat->ldo4 = Pmic_getBitField(regData, PMIC_LDO4_ILIM_ERR_SHIFT, PMIC_LDO4_ILIM_ERR_MASK);
        pPmicILIMStat->pldo1 = Pmic_getBitField(regData, PMIC_PLDO1_ILIM_ERR_SHIFT, PMIC_PLDO1_ILIM_ERR_MASK);
        pPmicILIMStat->pldo2 = Pmic_getBitField(regData, PMIC_PLDO2_ILIM_ERR_SHIFT, PMIC_PLDO2_ILIM_ERR_MASK);
        pPmicILIMStat->bbAvg = Pmic_getBitField(regData, PMIC_BB_AVG_ILIM_ERR_SHIFT, PMIC_BB_AVG_ILIM_ERR_MASK);
    }

    Pmic_criticalSectionStop(pPmicCoreHandle);

    return pmicStatus;
}
