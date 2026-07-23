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

/* ========================================================================== */
/*                             Include Files                                  */
/* ========================================================================== */
#include <stddef.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "pmic.h"
#include "pmic_common.h"

#include "pmic_fsm.h"
#include "pmic_io.h"
#include "regmap/fsm.h"

/* ========================================================================== */
/*                           Macros & Typedefs                                */
/* ========================================================================== */

/* In the register map, certain states are enumerated mulitple times
 * within the STATE bit field in the STATE_STAT register. This set of defines
 * is used to indicate the repeated bit fields. */
#define INIT_STATE_MIN      (1U)
#define INIT_STATE_MAX      (4U)
#define OFF_STATE_REPEATED  (0xEU)

/* ========================================================================== */
/*                        Interface Implementations                           */
/* ========================================================================== */
static inline void FSM_copyFsmCfg(const Pmic_FsmCfg_t *src, Pmic_FsmCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmCfg_t));
}

static inline void FSM_copyFsmWakeupCfg(const Pmic_FsmWakeupCfg_t *src, Pmic_FsmWakeupCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmWakeupCfg_t));
}

static inline void FSM_copyFsmWakeupStat(const Pmic_FsmWakeupStat_t *src, Pmic_FsmWakeupStat_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmWakeupStat_t));
}

static inline void FSM_copyFsmPwrLatchCfg(const Pmic_FsmPwrLatchCfg_t *src, Pmic_FsmPwrLatchCfg_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmPwrLatchCfg_t));
}

static inline void FSM_copyFsmPwrLatch(const Pmic_FsmPwrLatch_t *src, Pmic_FsmPwrLatch_t *dst) {
    (void)memmove((void *)dst, (const void *)src, sizeof(Pmic_FsmPwrLatch_t));
}

int32_t Pmic_fsmSetDevState(const Pmic_Handle_t *handle, uint8_t state)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (state > PMIC_STATE_REQUEST_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read STATE_CTRL
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, STATE_CTRL_REG, &regData);

        // Modify STATE_REQ and write STATE_CTRL
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, STATE_REQ_SHIFT, STATE_REQ_MASK, state);
            status = Pmic_ioTxByte(handle, STATE_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetDevState(const Pmic_Handle_t *handle, uint8_t *state)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (state == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read STATE_STAT
    if (status == PMIC_ST_SUCCESS)
    {
        status = Pmic_ioRxByte_CS(handle, STATE_STAT_REG, &regData);
    }

    // Extract STATE; account for repeated device states
    if (status == PMIC_ST_SUCCESS)
    {
        *state = Pmic_getBitField(regData, STATE_SHIFT, STATE_MASK);

        if ((*state >= INIT_STATE_MIN) && (*state <= INIT_STATE_MAX))
        {
            *state = PMIC_INIT_STATE;
        }

        if (*state == OFF_STATE_REPEATED)
        {
            *state = PMIC_OFF_STATE;
        }
    }

    return Pmic_logStatus(handle, status);
}

// Set FSM-related configurations in STBY_CFG register:
// 1. Pmic_FsmCfg_t.nrstActiveInStbySeq
// 2. Pmic_FsmCfg_t.stbyEn
// 3. Pmic_FsmCfg_t.higherVbatStbyExitThr
// 4. Pmic_FsmCfg_t.vbatStbyEntryThr
static int32_t FSM_setStbyCfg(const Pmic_Handle_t *handle, const Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID, status))
    {
        if (fsmCfg->vbatStbyEntryThr > PMIC_VBAT_STBY_ENTRY_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read STBY_CFG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, STBY_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify NRST_EN_STBY to be Pmic_FsmCfg_t.nrstActiveInStbySeq
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID))
            {
                Pmic_setBitField_b(&regData, NRST_EN_STBY_SHIFT, fsmCfg->nrstActiveInStbySeq);
            }

            // Modify STBY_EN to be Pmic_FsmCfg_t.stbyEn
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_STBY_EN_VALID))
            {
                Pmic_setBitField_b(&regData, STBY_EN_SHIFT, fsmCfg->stbyEn);
            }

            // Modify VBAT_STBY_EXIT_TH to be Pmic_FsmCfg_t.higherVbatStbyExitThr
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID))
            {
                Pmic_setBitField_b(&regData, VBAT_STBY_EXIT_TH_SHIFT, fsmCfg->higherVbatStbyExitThr);
            }

            // Modify VBAT_STBY_ENTRY_TH to be Pmic_FsmCfg_t.vbatStbyEntryThr
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID))
            {
                Pmic_setBitField(&regData, VBAT_STBY_ENTRY_TH_SHIFT, VBAT_STBY_ENTRY_TH_MASK,
                                 fsmCfg->vbatStbyEntryThr);
            }
        }

        // Write STBY_CFG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, STBY_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

// Set FSM-related configurations in SAFETY_CFG register:
// 1. Pmic_FsmCfg_t.autoBistEn
// 2. Pmic_FsmCfg_t.pwdThr
static int32_t FSM_setSafetyCfg(const Pmic_Handle_t *handle, const Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_PWD_THR_VALID, status))
    {
        if (fsmCfg->pwdThr > PMIC_PWD_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read SAFETY_CFG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, SAFETY_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify AUTO_BIST_EN to be Pmic_FsmCfg_t.autoBistEn
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_AUTO_BIST_EN_VALID))
            {
                Pmic_setBitField_b(&regData, AUTO_BIST_EN_SHIFT, fsmCfg->autoBistEn);
            }

            // Modify PWD_TH to be Pmic_FsmCfg_t.pwdThr
            if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_PWD_THR_VALID))
            {
                Pmic_setBitField(&regData, PWD_TH_SHIFT, PWD_TH_MASK, fsmCfg->pwdThr);
            }
        }

        // Write SAFETY_CFG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, SAFETY_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

// Set FSM-related configurations in RST_MCU_CFG register:
// 1. Pmic_FsmCfg_t.rstMcuTmo
// 2. Pmic_FsmCfg_t.nrstExt
static int32_t FSM_setRstMcuCfg(const Pmic_Handle_t *handle, const Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_RST_MCU_TMO_VALID, status))
    {
        if (fsmCfg->rstMcuTmo > PMIC_RST_MCU_TMO_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_NRST_EXT_VALID, status))
    {
        if (fsmCfg->nrstExt > PMIC_NRST_EXT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read RST_MCU_CFG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, RST_MCU_CFG_REG, &regData);

        // Modify RST_MCU_TMO_CFG to be Pmic_FsmCfg_t.rstMcuTmo
        if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_RST_MCU_TMO_VALID, status))
        {
            Pmic_setBitField(&regData, RST_MCU_TMO_CFG_SHIFT, RST_MCU_TMO_CFG_MASK, fsmCfg->rstMcuTmo);
        }

        // Modify NRST_EXT to be Pmic_FsmCfg_t.nrstExt
        if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_NRST_EXT_VALID, status))
        {
            Pmic_setBitField(&regData, NRST_EXT_SHIFT, NRST_EXT_MASK, fsmCfg->nrstExt);
        }

        // Write RST_MCU_CFG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, RST_MCU_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

// Set FSM-related configurations in SAFE_TMO_CFG register:
// 1. Pmic_FsmCfg_t.safeTmo
// 2. Pmic_FsmCfg_t.safeLockThr
static int32_t FSM_setSafeTmoCfg(const Pmic_Handle_t *handle, const Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_SAFE_TMO_VALID, status))
    {
        if (fsmCfg->safeTmo > PMIC_SAFE_TMO_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_SAFE_LOCK_THR_VALID, status))
    {
        if (fsmCfg->safeLockThr > PMIC_SAFE_LOCK_THR_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read SAFE_TMO_CFG_REG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, SAFE_TMO_CFG_REG, &regData);

        // Modify SAFE_TMO to be Pmic_FsmCfg_t.safeTmo
        if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_SAFE_TMO_VALID, status))
        {
            Pmic_setBitField(&regData, SAFE_TMO_SHIFT, SAFE_TMO_MASK, fsmCfg->safeTmo);
        }

        // Modify SAFE_LOCK_TH to be Pmic_FsmCfg_t.safeLockThr
        if (Pmic_validParamStatusCheck(fsmCfg->validParams, PMIC_CFG_SAFE_LOCK_THR_VALID, status))
        {
            Pmic_setBitField(&regData, SAFE_LOCK_TH_SHIFT, SAFE_LOCK_TH_MASK, fsmCfg->safeLockThr);
        }

        // Write SAFE_TMO_CFG_REG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, SAFE_TMO_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_fsmSetCfg(const Pmic_Handle_t *handle, const Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmCfg_t localFsmCfg;

    if ((status == PMIC_ST_SUCCESS) && (fsmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (fsmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmCfg(fsmCfg, &localFsmCfg);
    }

    // Set FSM-related configurations in STBY_CFG register:
    // 1. Pmic_FsmCfg_t.nrstActiveInStbySeq
    // 2. Pmic_FsmCfg_t.stbyEn
    // 3. Pmic_FsmCfg_t.higherVbatStbyExitThr
    // 4. Pmic_FsmCfg_t.vbatStbyEntryThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_STBY_EN_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID)))
    {
        status = FSM_setStbyCfg(handle, &localFsmCfg);
    }

    // Set FSM-related configurations in SAFETY_CFG register:
    // 1. Pmic_FsmCfg_t.autoBistEn
    // 2. Pmic_FsmCfg_t.pwdThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_AUTO_BIST_EN_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_PWD_THR_VALID)))
    {
        status = FSM_setSafetyCfg(handle, &localFsmCfg);
    }

    // Set FSM-related configurations in RST_MCU_CFG register:
    // 1. Pmic_FsmCfg_t.rstMcuTmo
    // 2. Pmic_FsmCfg_t.nrstExt
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_NRST_EXT_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_RST_MCU_TMO_VALID)))
    {
        status = FSM_setRstMcuCfg(handle, &localFsmCfg);
    }

    // Set FSM-related configurations in SAFE_TMO_CFG register:
    // 1. Pmic_FsmCfg_t.safeTmo
    // 2. Pmic_FsmCfg_t.safeLockThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_SAFE_TMO_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_SAFE_LOCK_THR_VALID)))
    {
        status = FSM_setSafeTmoCfg(handle, &localFsmCfg);
    }

    return Pmic_logStatus(handle, status);
}

// Get FSM-related configurations in STBY_CFG register:
// 1. Pmic_FsmCfg_t.nrstActiveInStbySeq
// 2. Pmic_FsmCfg_t.stbyEn
// 3. Pmic_FsmCfg_t.higherVbatStbyExitThr
// 4. Pmic_FsmCfg_t.vbatStbyEntryThr
static int32_t FSM_getStbyCfg(const Pmic_Handle_t *handle, Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read STBY_CFG
    status = Pmic_ioRxByte_CS(handle, STBY_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get Pmic_FsmCfg_t.nrstActiveInStbySeq
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID))
        {
            fsmCfg->nrstActiveInStbySeq = Pmic_getBitField_b(regData, NRST_EN_STBY_SHIFT);
        }

        // Get Pmic_FsmCfg_t.stbyEn
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_STBY_EN_VALID))
        {
            fsmCfg->stbyEn = Pmic_getBitField_b(regData, STBY_EN_SHIFT);
        }

        // Get Pmic_FsmCfg_t.higherVbatStbyExitThr
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID))
        {
            fsmCfg->higherVbatStbyExitThr = Pmic_getBitField_b(regData, VBAT_STBY_EXIT_TH_SHIFT);
        }

        // Get Pmic_FsmCfg_t.vbatStbyEntryThr
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID))
        {
            fsmCfg->vbatStbyEntryThr = Pmic_getBitField(regData, VBAT_STBY_ENTRY_TH_SHIFT, VBAT_STBY_ENTRY_TH_MASK);
        }
    }

    return status;
}

// Get FSM-related configurations in SAFETY_CFG register:
// 1. Pmic_FsmCfg_t.autoBistEn
// 2. Pmic_FsmCfg_t.pwdThr
static int32_t FSM_getSafetyCfg(const Pmic_Handle_t *handle, Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read SAFETY_CFG
    status = Pmic_ioRxByte_CS(handle, SAFETY_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract AUTO_BIST_EN
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_AUTO_BIST_EN_VALID))
        {
            fsmCfg->autoBistEn = Pmic_getBitField_b(regData, AUTO_BIST_EN_SHIFT);
        }

        // Extract PWD_TH
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_PWD_THR_VALID))
        {
            fsmCfg->pwdThr = Pmic_getBitField(regData, PWD_TH_SHIFT, PWD_TH_MASK);
        }
    }

    return status;
}

// Get FSM-related configurations in RST_MCU_CFG register:
// 1. Pmic_FsmCfg_t.rstMcuTmo
// 2. Pmic_FsmCfg_t.nrstExt
static int32_t FSM_getRstMcuCfg(const Pmic_Handle_t *handle, Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read RST_MCU_CFG
    status = Pmic_ioRxByte_CS(handle, RST_MCU_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract RST_MCU_TMO_CFG
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_RST_MCU_TMO_VALID))
        {
            fsmCfg->rstMcuTmo = Pmic_getBitField(regData, RST_MCU_TMO_CFG_SHIFT, RST_MCU_TMO_CFG_MASK);
        }

        // Extract NRST_EXT
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_NRST_EXT_VALID))
        {
            fsmCfg->nrstExt = Pmic_getBitField(regData, NRST_EXT_SHIFT, NRST_EXT_MASK);
        }
    }

    return status;
}

// Get FSM-related configurations in SAFE_TMO_CFG register:
// 1. Pmic_FsmCfg_t.safeTmo
// 2. Pmic_FsmCfg_t.safeLockThr
static int32_t FSM_getSafeTmoCfg(const Pmic_Handle_t *handle, Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read SAFE_TMO_CFG_REG
    status = Pmic_ioRxByte_CS(handle, SAFE_TMO_CFG_REG, &regData);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract SAFE_TMO
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_SAFE_TMO_VALID))
        {
            fsmCfg->safeTmo = Pmic_getBitField(regData, SAFE_TMO_SHIFT, SAFE_TMO_MASK);
        }

        // Extract SAFE_LOCK_TH
        if (Pmic_validParamCheck(fsmCfg->validParams, PMIC_CFG_SAFE_LOCK_THR_VALID))
        {
            fsmCfg->safeLockThr = Pmic_getBitField(regData, SAFE_LOCK_TH_SHIFT, SAFE_LOCK_TH_MASK);
        }
    }

    return status;
}

int32_t Pmic_fsmGetCfg(const Pmic_Handle_t *handle, Pmic_FsmCfg_t *fsmCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmCfg_t localFsmCfg;

    if ((status == PMIC_ST_SUCCESS) && (fsmCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (fsmCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmCfg(fsmCfg, &localFsmCfg);
    }

    // Get FSM-related configurations in STBY_CFG register:
    // 1. Pmic_FsmCfg_t.nrstActiveInStbySeq
    // 2. Pmic_FsmCfg_t.stbyEn
    // 3. Pmic_FsmCfg_t.higherVbatStbyExitThr
    // 4. Pmic_FsmCfg_t.vbatStbyEntryThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_NRST_ACTIVE_IN_STBY_SEQ_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_STBY_EN_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_HIGHER_VBAT_STBY_EXIT_THR_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_VBAT_STBY_ENTRY_THR_VALID)))
    {
        status = FSM_getStbyCfg(handle, &localFsmCfg);
    }

    // Get FSM-related configurations in SAFETY_CFG register:
    // 1. Pmic_FsmCfg_t.autoBistEn
    // 2. Pmic_FsmCfg_t.pwdThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_AUTO_BIST_EN_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_PWD_THR_VALID)))
    {
        status = FSM_getSafetyCfg(handle, &localFsmCfg);
    }

    // Get FSM-related configurations in RST_MCU_CFG register:
    // 1. Pmic_FsmCfg_t.rstMcuTmo
    // 2. Pmic_FsmCfg_t.nrstExt
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_NRST_EXT_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_RST_MCU_TMO_VALID)))
    {
        status = FSM_getRstMcuCfg(handle, &localFsmCfg);
    }

    // Get FSM-related configurations in SAFE_TMO_CFG register:
    // 1. Pmic_FsmCfg_t.safeTmo
    // 2. Pmic_FsmCfg_t.safeLockThr
    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_SAFE_TMO_VALID) ||
         Pmic_validParamCheck(localFsmCfg.validParams, PMIC_CFG_SAFE_LOCK_THR_VALID)))
    {
        status = FSM_getSafeTmoCfg(handle, &localFsmCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmCfg(&localFsmCfg, fsmCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetDevErrCnt(const Pmic_Handle_t *handle, uint8_t devErrCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (devErrCnt > PMIC_DEV_ERR_CNT_MAX))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read DEV_ERR_STAT
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, DEV_ERR_STAT_REG, &regData);

        // Extract DEV_ERR_CNT and write DEV_ERR_STAT
        if (status == PMIC_ST_SUCCESS)
        {
            Pmic_setBitField(&regData, DEV_ERR_CNT_SHIFT, DEV_ERR_CNT_MASK, devErrCnt);
            status = Pmic_ioTxByte(handle, DEV_ERR_STAT_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetDevErrCnt(const Pmic_Handle_t *handle, uint8_t *devErrCnt)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (devErrCnt == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read DEV_ERR_STAT
        status = Pmic_ioRxByte_CS(handle, DEV_ERR_STAT_REG, &regData);

        // Extract DEV_ERR_CNT
        if (status == PMIC_ST_SUCCESS)
        {
            *devErrCnt = Pmic_getBitField(regData, DEV_ERR_CNT_SHIFT, DEV_ERR_CNT_MASK);
        }
    }

    return Pmic_logStatus(handle, status);
}

static int32_t FSM_validateWakeupCfg(const Pmic_FsmWakeupCfg_t *wakeupCfg)
{
    int32_t status = PMIC_ST_SUCCESS;

    if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_EVENT_VALID, status))
    {
        if (wakeupCfg->wake1Event > PMIC_WAKE_EVENT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_EVENT_VALID, status))
    {
        if (wakeupCfg->wake2Event > PMIC_WAKE_EVENT_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_DGL_VALID, status))
    {
        if (wakeupCfg->wake1Dgl > PMIC_WAKE_DEGLITCH_TIME_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_DGL_VALID, status))
    {
        if (wakeupCfg->wake2Dgl > PMIC_WAKE_DEGLITCH_TIME_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    return status;
}

static int32_t FSM_setWakeupCfg(const Pmic_Handle_t *handle, const Pmic_FsmWakeupCfg_t *wakeupCfg)
{
    int32_t status = FSM_validateWakeupCfg(wakeupCfg);
    uint8_t regData = 0U;

    if (status == PMIC_ST_SUCCESS)
    {
        // Read WAKE_CFG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, WAKE_CFG_REG, &regData);

        // Modify WAKE1_STBY_CFG to be Pmic_FsmWakeupCfg_t.wake1Event
        if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_EVENT_VALID, status))
        {
            Pmic_setBitField(&regData, WAKE1_STBY_CFG_SHIFT, WAKE1_STBY_CFG_MASK, wakeupCfg->wake1Event);
        }

        // Modify WAKE2_STBY_CFG to be Pmic_FsmWakeupCfg_t.wake2Event
        if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_EVENT_VALID, status))
        {
            Pmic_setBitField(&regData, WAKE2_STBY_CFG_SHIFT, WAKE2_STBY_CFG_MASK, wakeupCfg->wake2Event);
        }

        // Modify WAKE1_DGL_CFG to be Pmic_FsmWakeupCfg_t.wake1Dgl
        if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_DGL_VALID, status))
        {
            Pmic_setBitField(&regData, WAKE1_DGL_CFG_SHIFT, WAKE1_DGL_CFG_MASK, wakeupCfg->wake1Dgl);
        }

        // Modify WAKE2_DGL_CFG to be Pmic_FsmWakeupCfg_t.wake2Dgl
        if (Pmic_validParamStatusCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_DGL_VALID, status))
        {
            Pmic_setBitField(&regData, WAKE2_DGL_CFG_SHIFT, WAKE2_DGL_CFG_MASK, wakeupCfg->wake2Dgl);
        }

        // Write WAKE_CFG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, WAKE_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_fsmSetWakeupCfg(const Pmic_Handle_t *handle, const Pmic_FsmWakeupCfg_t *wakeupCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmWakeupCfg_t localWakeupCfg;

    if ((status == PMIC_ST_SUCCESS) && (wakeupCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (wakeupCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmWakeupCfg(wakeupCfg, &localWakeupCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE1_EVENT_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE2_EVENT_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE1_DGL_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE2_DGL_VALID)))
    {
        status = FSM_setWakeupCfg(handle, &localWakeupCfg);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t FSM_getWakeupCfg(const Pmic_Handle_t *handle, Pmic_FsmWakeupCfg_t *wakeupCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read WAKE_CFG
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, WAKE_CFG_REG, &regData);
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        // Extract WAKE1_STBY_CFG
        if (Pmic_validParamCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_EVENT_VALID))
        {
            wakeupCfg->wake1Event = Pmic_getBitField(regData, WAKE1_STBY_CFG_SHIFT, WAKE1_STBY_CFG_MASK);
        }

        // Extract WAKE2_STBY_CFG
        if (Pmic_validParamCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_EVENT_VALID))
        {
            wakeupCfg->wake2Event = Pmic_getBitField(regData, WAKE2_STBY_CFG_SHIFT, WAKE2_STBY_CFG_MASK);
        }

        // Extract WAKE1_DGL_CFG
        if (Pmic_validParamCheck(wakeupCfg->validParams, PMIC_CFG_WAKE1_DGL_VALID))
        {
            wakeupCfg->wake1Dgl = Pmic_getBitField(regData, WAKE1_DGL_CFG_SHIFT, WAKE1_DGL_CFG_MASK);
        }

        // Extract WAKE2_DGL_CFG
        if (Pmic_validParamCheck(wakeupCfg->validParams, PMIC_CFG_WAKE2_DGL_VALID))
        {
            wakeupCfg->wake2Dgl = Pmic_getBitField(regData, WAKE2_DGL_CFG_SHIFT, WAKE2_DGL_CFG_MASK);
        }
    }

    return status;
}

int32_t Pmic_fsmGetWakeupCfg(const Pmic_Handle_t *handle, Pmic_FsmWakeupCfg_t *wakeupCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmWakeupCfg_t localWakeupCfg;

    if ((status == PMIC_ST_SUCCESS) && (wakeupCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (wakeupCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmWakeupCfg(wakeupCfg, &localWakeupCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE1_EVENT_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE2_EVENT_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE1_DGL_VALID) ||
         Pmic_validParamCheck(localWakeupCfg.validParams, PMIC_CFG_WAKE2_DGL_VALID)))
    {
        status = FSM_getWakeupCfg(handle, &localWakeupCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmWakeupCfg(&localWakeupCfg, wakeupCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetWakeStatus(const Pmic_Handle_t *handle, Pmic_FsmWakeupStat_t *wakeupStat)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    Pmic_FsmWakeupStat_t localWakeupStat;

    if ((status == PMIC_ST_SUCCESS) && (wakeupStat == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmWakeupStat(wakeupStat, &localWakeupStat);
    }

    // Read WAKE_STAT
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, WAKE_STAT_REG, &regData);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Extract WAKE1_LVL_DGL, WAKE2_LVL_DGL, WAKE_SOURCE
    if (status == PMIC_ST_SUCCESS)
    {
        localWakeupStat.wake1Lvl = Pmic_getBitField_b(regData, WAKE1_LVL_DGL_SHIFT);
        localWakeupStat.wake2Lvl = Pmic_getBitField_b(regData, WAKE2_LVL_DGL_SHIFT);
        localWakeupStat.wakeSrc = Pmic_getBitField(regData, WAKE_SOURCE_SHIFT, WAKE_SOURCE_MASK);
        FSM_copyFsmWakeupStat(&localWakeupStat, wakeupStat);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t FSM_setPwrLatchCfg(const Pmic_Handle_t *handle, const Pmic_FsmPwrLatchCfg_t *pwrLatchCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    if (Pmic_validParamStatusCheck(pwrLatchCfg->validParams, PMIC_CFG_PWD_DLY_VALID, status))
    {
        if (pwrLatchCfg->pwdDly > PMIC_PWD_DLY_MAX)
        {
            status = PMIC_ST_ERR_INV_PARAM;
        }
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read PWRL_CFG
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PWRL_CFG_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify STBY_ERR_WK_PWRL_EN
            if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID))
            {
                Pmic_setBitField_b(&regData, STBY_ERR_WK_PWRL_EN_SHIFT, pwrLatchCfg->stbyErrWakeEventPwrlEn);
            }

            // Modify WAKE2_PWRL_EN
            if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID))
            {
                Pmic_setBitField_b(&regData, WAKE2_PWRL_EN_SHIFT, pwrLatchCfg->wake2EventPwrlEn);
            }

            // Modify WAKE1_PWRL_EN
            if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID))
            {
                Pmic_setBitField_b(&regData, WAKE1_PWRL_EN_SHIFT, pwrLatchCfg->wake1EventPwrlEn);
            }

            // Modify PWRD_DLY_CFG
            if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_PWD_DLY_VALID))
            {
                Pmic_setBitField(&regData, PWRD_DLY_CFG_SHIFT, PWRD_DLY_CFG_MASK, pwrLatchCfg->pwdDly);
            }
        }

        // Write PWRL_CFG
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, PWRL_CFG_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return status;
}

int32_t Pmic_fsmSetPowerLatchCfg(const Pmic_Handle_t *handle, const Pmic_FsmPwrLatchCfg_t *pwrLatchCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmPwrLatchCfg_t localPwrLatchCfg;

    if ((status == PMIC_ST_SUCCESS) && (pwrLatchCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pwrLatchCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmPwrLatchCfg(pwrLatchCfg, &localPwrLatchCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_PWD_DLY_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID)))
    {
        status = FSM_setPwrLatchCfg(handle, &localPwrLatchCfg);
    }

    return Pmic_logStatus(handle, status);
}

static int32_t FSM_getPwrLatchCfg(const Pmic_Handle_t *handle, Pmic_FsmPwrLatchCfg_t *pwrLatchCfg)
{
    int32_t status = PMIC_ST_SUCCESS;
    uint8_t regData = 0U;

    // Read PWRL_CFG
    Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
    status = Pmic_ioRxByte(handle, PWRL_CFG_REG, &regData);
    Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);

    if (status == PMIC_ST_SUCCESS)
    {
        // Get STBY_ERR_WK_PWRL_EN
        if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID))
        {
            pwrLatchCfg->stbyErrWakeEventPwrlEn = Pmic_getBitField_b(regData, STBY_ERR_WK_PWRL_EN_SHIFT);
        }

        // Get WAKE2_PWRL_EN
        if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID))
        {
            pwrLatchCfg->wake2EventPwrlEn = Pmic_getBitField_b(regData, WAKE2_PWRL_EN_SHIFT);
        }

        // Get WAKE1_PWRL_EN
        if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID))
        {
            pwrLatchCfg->wake1EventPwrlEn = Pmic_getBitField_b(regData, WAKE1_PWRL_EN_SHIFT);
        }

        // Get PWRD_DLY_CFG
        if (Pmic_validParamCheck(pwrLatchCfg->validParams, PMIC_CFG_PWD_DLY_VALID))
        {
            pwrLatchCfg->pwdDly = Pmic_getBitField(regData, PWRD_DLY_CFG_SHIFT, PWRD_DLY_CFG_MASK);
        }
    }

    return status;
}

int32_t Pmic_fsmGetPowerLatchCfg(const Pmic_Handle_t *handle, Pmic_FsmPwrLatchCfg_t *pwrLatchCfg)
{
    int32_t status = Pmic_checkHandle(handle);
    Pmic_FsmPwrLatchCfg_t localPwrLatchCfg;

    if ((status == PMIC_ST_SUCCESS) && (pwrLatchCfg == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pwrLatchCfg->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmPwrLatchCfg(pwrLatchCfg, &localPwrLatchCfg);
    }

    if ((status == PMIC_ST_SUCCESS) &&
        (Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_PWD_DLY_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_STBY_ERR_WAKE_EVENT_PWRL_EN_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_WAKE1_EVENT_PWRL_EN_VALID) ||
         Pmic_validParamCheck(localPwrLatchCfg.validParams, PMIC_CFG_WAKE2_EVENT_PWRL_EN_VALID)))
    {
        status = FSM_getPwrLatchCfg(handle, &localPwrLatchCfg);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmPwrLatchCfg(&localPwrLatchCfg, pwrLatchCfg);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmSetPowerLatch(const Pmic_Handle_t *handle, const Pmic_FsmPwrLatch_t *pwrLatch)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    Pmic_FsmPwrLatch_t localPwrLatch;

    if ((status == PMIC_ST_SUCCESS) && (pwrLatch == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pwrLatch->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmPwrLatch(pwrLatch, &localPwrLatch);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Read PWRL_CTRL
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PWRL_CTRL_REG, &regData);

        if (status == PMIC_ST_SUCCESS)
        {
            // Modify STBY_ERR_WK_PWRL
            if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID))
            {
                Pmic_setBitField_b(&regData, STBY_ERR_WK_PWRL_SHIFT, localPwrLatch.stbyErrWakeLatch);
            }

            // Modify TMR_WK_PWRL
            if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_STBY_TMR_WAKE_LATCH_VALID))
            {
                Pmic_setBitField_b(&regData, TMR_WK_PWRL_SHIFT, localPwrLatch.stbyTmrWakeLatch);
            }

            // Modify M_PMIC_WK_PWRL
            if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_M_PMIC_WAKE_LATCH_VALID))
            {
                Pmic_setBitField_b(&regData, M_PMIC_WK_PWRL_SHIFT, localPwrLatch.mPmicWakeLatch);
            }

            // Modify WAKE2_PWRL
            if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_WAKE1_LATCH_VALID))
            {
                Pmic_setBitField_b(&regData, WAKE2_PWRL_SHIFT, localPwrLatch.wake2Latch);
            }

            // Modify WAKE1_PWRL
            if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_WAKE2_LATCH_VALID))
            {
                Pmic_setBitField_b(&regData, WAKE1_PWRL_SHIFT, localPwrLatch.wake1Latch);
            }
        }

        // Write PWRL_CTRL
        if (status == PMIC_ST_SUCCESS)
        {
            status = Pmic_ioTxByte(handle, PWRL_CTRL_REG, regData);
        }
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetPowerLatch(const Pmic_Handle_t *handle, Pmic_FsmPwrLatch_t *pwrLatch)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;
    Pmic_FsmPwrLatch_t localPwrLatch;

    if ((status == PMIC_ST_SUCCESS) && (pwrLatch == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    if ((status == PMIC_ST_SUCCESS) && (pwrLatch->validParams == 0U))
    {
        status = PMIC_ST_ERR_INV_PARAM;
    }

    if (status == PMIC_ST_SUCCESS)
    {
        FSM_copyFsmPwrLatch(pwrLatch, &localPwrLatch);
    }

    // Read PWRL_CTRL
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, PWRL_CTRL_REG, &regData);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    if (status == PMIC_ST_SUCCESS)
    {
        // Get STBY_ERR_WK_PWRL
        if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_STBY_ERR_WAKE_LATCH_VALID))
        {
            localPwrLatch.stbyErrWakeLatch = Pmic_getBitField_b(regData, STBY_ERR_WK_PWRL_SHIFT);
        }

        // Get TMR_WK_PWRL
        if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_STBY_TMR_WAKE_LATCH_VALID))
        {
            localPwrLatch.stbyTmrWakeLatch = Pmic_getBitField_b(regData, TMR_WK_PWRL_SHIFT);
        }

        // Get M_PMIC_WK_PWRL
        if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_M_PMIC_WAKE_LATCH_VALID))
        {
            localPwrLatch.mPmicWakeLatch = Pmic_getBitField_b(regData, M_PMIC_WK_PWRL_SHIFT);
        }

        // Get WAKE2_PWRL
        if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_WAKE1_LATCH_VALID))
        {
            localPwrLatch.wake2Latch = Pmic_getBitField_b(regData, WAKE2_PWRL_SHIFT);
        }

        // Get WAKE1_PWRL
        if (Pmic_validParamCheck(localPwrLatch.validParams, PMIC_CFG_WAKE2_LATCH_VALID))
        {
            localPwrLatch.wake1Latch = Pmic_getBitField_b(regData, WAKE1_PWRL_SHIFT);
        }

        FSM_copyFsmPwrLatch(&localPwrLatch, pwrLatch);
    }

    return Pmic_logStatus(handle, status);
}

int32_t Pmic_fsmGetLastResetMcuStateDuration(const Pmic_Handle_t *handle, uint8_t *duration)
{
    int32_t status = Pmic_checkHandle(handle);
    uint8_t regData = 0U;

    if ((status == PMIC_ST_SUCCESS) && (duration == NULL))
    {
        status = PMIC_ST_ERR_NULL_PARAM;
    }

    // Read RST_MCU_TMR_REG
    if (status == PMIC_ST_SUCCESS)
    {
        Pmic_criticalSectionStart(handle, PMIC_COMMUNICATION);
        status = Pmic_ioRxByte(handle, RST_MCU_TMR_REG_REG, &regData);
        Pmic_criticalSectionStop(handle, PMIC_COMMUNICATION);
    }

    // Extract RST_MCU_TMR
    if (status == PMIC_ST_SUCCESS)
    {
        *duration = Pmic_getBitField(regData, RST_MCU_TMR_SHIFT, RST_MCU_TMR_MASK);
    }

    return Pmic_logStatus(handle, status);
}
